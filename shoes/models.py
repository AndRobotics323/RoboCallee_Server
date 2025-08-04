
from django.db import models



# Create your models here.


class Product(models.Model):

    modify_date = models.DateTimeField(null=True, blank=True)

    model = models.CharField(max_length=200)
    size = models.PositiveIntegerField()
    color = models.CharField(max_length=200)


    x = models.FloatField(default=0.0)  # 🔥 x 좌표
    y = models.FloatField(default=0.0)  # 🔥 y 좌표


    photo = models.ImageField(upload_to='product_images/', null=True, blank=True)
    description = models.TextField()
    create_date = models.DateTimeField()
    
    price = models.PositiveIntegerField()
    stock = models.PositiveIntegerField(default=0)  # 🔥 재고 수량

    def __str__(self):
        return f"{self.model} {self.size} (재고: {self.stock})"


    # modify_date = models.DateTimeField(null=True, blank=True)

    # def __str__(self):
    #     return self.subject
